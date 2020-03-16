"use strict";
import React from 'react';
// import * as FontAwesome from 'react-icons/lib/fa'
import {Card, TableRow, TableColumn, FontIcon, Button, Icon, Text } from 'react-md';
import { Link } from 'react-router-dom';
import { SimpleLink } from './SimpleLink';
import StarRatingComponent from "react-star-rating-component"
import UserEvaluationService from '../services/UserEvaluationService';

import AuthService from '../services/AuthService';

export class SearchListRow extends React.Component {

    constructor(props) {
        super(props);
        this.state ={
            avg: 0
        }
    }

    componentWillMount(){
        UserEvaluationService.getUserEvaluation(this.props.items.owner_id).then((user_evals) => {
            console.log("evals",user_evals);
            var sum = 0;
            for( var i = 0; i < user_evals.length; i++ ){
                sum += parseInt( user_evals[i].rating, 10 ); //don't forget to add the base
            }
            var avg = 0;
            if(user_evals.length > 0){
                avg = sum/user_evals.length;
            }
            avg = 2* avg; // scale from 0-10
            this.setState({
                avg: avg,
            });
            console.log(this.state)
        }).catch((e) => {
            console.error(e);
        });

    }
    render() {
        return (
            <div className="mdc-card" style={{margin:"2em"}}>
                <div className="mdc-card__media mdc-card__media--square">
                    <div className="mdc-card__media-content">
                        <SimpleLink className="card_link" to={`/item/${this.props.items._id}`}>{this.props.items.name}</SimpleLink>
                    </div>
                    <div className="mdc-card__media-content">
                        <SimpleLink to={`/item/${this.props.items._id}`}>
                            <img className="img-fluid" height={300} width={300} src = {`http://localhost:3000/photos/${this.props.items.photos[0]}`}/>
                        </SimpleLink>
                    </div>
                </div>
                <div className="mdc-card__media mdc-card__media--square custom">
                    <StarRatingComponent
                        name="rate1"
                        starCount={10}
                        value={this.state.avg}
                    />
                </div>
                <div className="mdc-card__action-buttons customSearch">
                    {this.props.items.description}
                </div>
                <div className="mdc-card__action-buttons customSearch">
                    Price per day of leasing:{this.props.items.price}
                </div>
            </div>
            // {/*<TableRow key={this.props.key}>*/}
            //     <TableColumn><Link to={`/item/${this.props.items._id}`}><FontIcon>image</FontIcon></Link></TableColumn>
            //     <TableColumn><SimpleLink to={`/item/${this.props.items._id}`}>{this.props.items.name}</SimpleLink></TableColumn>
            // </TableRow>
        );
    }
}