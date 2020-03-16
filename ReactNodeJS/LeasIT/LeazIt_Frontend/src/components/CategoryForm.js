"use strict";

import React from 'react';
import { TextField } from 'react-md';
import { withRouter } from 'react-router-dom'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import Page from './Page';

import { AlertMessage } from './AlertMessage';
import {
    Badge,
    Button,
    ButtonDropdown,
    ButtonGroup,
    ButtonToolbar,
    Card,
    CardBody,
    CardFooter,
    CardHeader,
    CardTitle,
    Col,
    Dropdown,
    DropdownItem,
    FormGroup,
    Form,
    FormText,
    Row,
    Input,
    Label,
    Table
} from 'reactstrap';


const style = { maxWidth: 500 };


class CategoryForm extends React.Component {

    constructor(props) {
        super(props);
        console.log("CategoryForm",this.props);
        if(this.props.categories != undefined) {
            this.state = {
                name : props.categories.name,
            };
        } else {
            this.state = {
                name : '',
            };
        }

        this.handleChangeName = this.handleChangeName.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChangeName(event) {
        this.setState(Object.assign({}, this.state, {name: event.target.value}));
        console.log(this.state.name);
    }

    handleSubmit(event) {
        event.preventDefault();

        let category = this.props.categories;
        if(category === undefined) {
            category = {};
        }
        if(this.state.name === undefined){
            return
        }

        category.name = this.state.name;
        console.log(this.props);
        this.props.onSubmit(category);
    }


    render() {
        let categories = this.props.data;
        let self = this.props;
        console.log('lathgories: ', this.props);
        return (
            <Page>
                <div className="animated fadeIn">
                    <Row>
                        <Col xs="12" md="6">
                            <Card>
                                {/*<CardHeader>*/}
                                {/*LeazIt Users*/}
                                {/*</CardHeader>*/}
                                <CardBody>

                                    <Table hover responsive className="table-outline mb-0 d-none d-sm-table">
                                        <thead className="thead-light">
                                        <tr>
                                            <th className="text-center"><FontAwesomeIcon icon="clipboard-list" /></th>
                                            <th>Categories</th>
                                            <th/>
                                        </tr>
                                        </thead>

                                        <tbody>
                                        {Object.keys(categories).map(function(key) {
                                                return(

                                                    <tr>
                                                        <td className="text-center">
                                                        </td>
                                                        <td>
                                                            <div>
                                                               {categories[key].name}
                                                            </div>
                                                            <div className="small text-muted">
                                                                {/*<span>{data[key].first_name}</span>|        {data[key].last_name}*/}
                                                            </div>
                                                        </td>
                                                        <td className="text-center">
                                                            <a className="btn btn-danger" onClick={(id) => self.onDelete(categories[key]._id)} ><FontAwesomeIcon color={'white'} icon="trash-alt"/></a>
                                                        </td>
                                                    </tr>
                                                )})}
                                        </tbody>
                                    </Table>
                                </CardBody>
                            </Card>
                        </Col>
                        <Col xs="12" md="6">
                            <Card>
                                <CardHeader>
                                    <strong>Add a category</strong>
                                </CardHeader>
                                <CardBody>
                                    <Form onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()} className="form-horizontal">
                                        <FormGroup row>
                                            {/*<Col md="6">*/}
                                                {/*/!*<Label htmlFor="text"></Label>*!/*/}
                                            {/*</Col>*/}
                                            <Col xs="12" md="9">
                                                <input type="text" id="CategoryInput" placeholder="Category" required value={this.state.name} onChange={this.handleChangeName}/>
                                                <FormText className="help-block"><span>Enter a category here e.g. Hair Dyer, Hammer, etc</span></FormText>
                                            </Col>
                                        </FormGroup>

                                        <Row>
                                        <Col md="6">
                                            <Button id="submit" type="submit" size='sm' color="primary" raised><i className="fa fa-dot-circle-o"/> Submit</Button>
                                        </Col>
                                        <Col md="6">
                                            <Button id="reset" type="reset" size="sm" color="danger" raised><i className="fa fa-ban"/> Cancel</Button>
                                        </Col>
                                        </Row>
                                    </Form>
                                </CardBody>
                            </Card>
                        </Col>
                    </Row>
                </div>
            </Page>
            // <Page>
            //     <Card style={style} className="md-block-centered">
            //         <form className="md-grid" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
            //             <TextField
            //                 label="name"
            //                 id="TitleField"
            //                 type="text"
            //                 className="md-row"
            //                 required={true}
            //                 value={this.state.name}
            //                 onChange={this.handleChangeName}
            //                 errorText="Title is required"/>
            //             <Button id="submit" type="submit"
            //                     raised primary className="md-cell md-cell--2">Save</Button>
            //             <Button id="reset" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
            //             <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
            //         </form>
            //     </Card>
            // </Page>
        );
    }
}

export default withRouter(CategoryForm);