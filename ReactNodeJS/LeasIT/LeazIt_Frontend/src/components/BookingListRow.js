"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';


export class BookingListRow extends React.Component {

    constructor(props) {
        super(props);
        console.log(props);
    }

    render() {
        if(this.props.itemId === undefined) {
            return (
                <TableRow key={this.props.key}>

                    <TableColumn>
                        <Link
                            to={`/items/${this.props.booking.item_id._id}`}><img src= {`http://localhost:3000/photos/${this.props.booking.item_id.photos}`} className="message-photo" alt=''/></Link>
                    </TableColumn>
                    <TableColumn>
                        <Link
                            to={`/items/${this.props.booking.item_id._id}`} style={{color: 'black'}}>{this.props.booking.item_id.name}</Link>
                    </TableColumn>
                    <TableColumn>
                        <Link
                            to={`/items/${this.props.booking.item_id._id}/booking/${this.props.booking._id}`} style={{color: 'black'}}>{this.props.booking.startDate}</Link>
                    </TableColumn>
                    <TableColumn>
                        <Link
                            to={`/items/${this.props.booking.item_id._id}/booking/${this.props.booking._id}`} style={{color: 'black'}}>{this.props.booking.endDate}</Link>
                    </TableColumn>
                    <TableColumn className="text-center">
                        <a className="btn btn-info lg"><Link
                            to={`/items/${this.props.booking.item_id._id}/booking/${this.props.booking._id}`} style={{color: 'white'}}>View</Link></a>

                    </TableColumn>
                </TableRow>

                //
                // <TableRow key={this.props.key}>
                //     <TableColumn>  <Link
                //         to={`/booking/${this.props.booking._id}`}><FontIcon>image</FontIcon></Link></TableColumn>
                //     <TableColumn><SimpleLink
                //         to={`/booking/${this.props.booking._id}`}>{this.props.booking._id}</SimpleLink></TableColumn>
                //     {AuthService.isAuthenticated() ?
                //         <TableColumn><Link
                //             to={`/booking/${this.props.booking._id}`}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                //         : <TableColumn><Link to={'/login'}><FontIcon>mode_edit</FontIcon></Link></TableColumn>
                //     }
                //     {AuthService.isAuthenticated() ?
                //         <TableColumn><Button onClick={() => this.props.onDelete(this.props.booking._id)}
                //                              icon>delete</Button></TableColumn>
                //         : <TableColumn><Link to={'/login'}><FontIcon>delete</FontIcon></Link></TableColumn>
                //     }
                //     <TableColumn><Link
                //     to={`/create_evaluation/`}><FontIcon>half-star-alt</FontIcon></Link></TableColumn>
                //
                // </TableRow>


            );
        }
        else{
            console.log("in4",`/items/${this.props.itemId}/booking/${this.props.booking._id}`);
            return (
                <TableRow key={this.props.key}>
                    {/*<TableColumn>*/}
                        {/*<Link*/}
                        {/*to={`/items/${this.props.itemId}/booking/${this.props.booking._id}`}><img src= {`http://localhost:3000/photos/${this.props.booking.booker_id.photo}`} className="message-photo" alt=''/></Link>*/}
                    {/*</TableColumn>                    */}
                    <TableColumn>
                        <Link
                        to={`/profile/${this.props.booking.booker_id._id}`}><img src= {`http://localhost:3000/photos/${this.props.booking.booker_id.photo}`} className="message-photo" alt=''/></Link>
                    </TableColumn>
                    <TableColumn>
                        <Link
                            to={`/profile/${this.props.booking.booker_id._id}`} style={{color: 'black'}}>{this.props.booking.booker_id.username}</Link>
                    </TableColumn>
                    <TableColumn>
                        <Link
                            to={`/items/${this.props.booking.item_id._id}/booking/${this.props.booking._id}`} style={{color: 'black'}}>{this.props.booking.startDate}</Link>
                    </TableColumn>
                    <TableColumn>
                        <Link
                            to={`/items/${this.props.booking.item_id._id}/booking/${this.props.booking._id}`} style={{color: 'black'}}>{this.props.booking.endDate}</Link>
                    </TableColumn>
                    <TableColumn className="text-center">
                        <a className="btn btn-info lg"><Link
                            to={`/items/${this.props.booking.item_id._id}/booking/${this.props.booking._id}`} style={{color: 'white'}}>View</Link></a>

                    </TableColumn>




                </TableRow>
            );

        }
    }
}